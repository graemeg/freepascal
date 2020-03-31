program fppkg;

{$mode objfpc}{$H+}{$macro on}

{$if defined(VER2_2) and (FPC_PATCH<1)}
  {$fatal At least FPC 2.2.1 is required to compile fppkg}
{$endif}

{$ifndef package_version_major}
  {$define package_version_major:=0}
{$endif}
{$ifndef package_version_minor}
  {$define package_version_minor:=0}
{$endif}
{$ifndef package_version_micro}
  {$define package_version_micro:=0}
{$endif}
{$ifndef package_version_build}
  {$define package_version_build:=0}
{$endif}

uses
  // General
{$ifdef unix}
  baseunix, cthreads,
{$endif}
  Classes, SysUtils, TypInfo, custapp, inifiles,
  // Repository handler objects
  fprepos, fpxmlrep,
  pkgmessages, pkgglobals, pkgoptions, pkgrepos,
  // Package Handler components
  pkghandler,pkgmkconv, pkgdownload,
  pkgfpmake, pkgcommands,
  pkgPackagesStructure,
  fpmkunit
  // Downloaders
{$if (defined(unix) and not defined(android)) or defined(windows)}
  ,pkgwget
  ,pkglnet
  ,pkgfphttp
  ,opensslsockets
{$endif}
  ;

const
  version_major = package_version_major;
  version_minor = package_version_minor;
  version_micro = package_version_micro;
  version_build = package_version_build;

Type
  { TMakeTool }

  TMakeTool = Class(TCustomApplication)
  Private
    ParaAction   : string;
    ParaPackages : TStringList;
    procedure HandleConfig;
    procedure MaybeCreateLocalDirs;
    procedure ShowUsage(const aErrorMsg : String = '');
    procedure ShowVersion;
  Public
    Constructor Create; overload;
    Constructor Create(aOwner : TComponent); overload;  override;
    Destructor Destroy;override;
    Procedure LoadGlobalDefaults;
    Procedure ProcessCommandLine(FirstPass: boolean);
    Procedure DoRun; Override;
  end;

  EMakeToolError = Class(Exception);


{ TMakeTool }

procedure TMakeTool.LoadGlobalDefaults;
var
  i : integer;
  cfgfile : String;
begin
  // Default verbosity
  LogLevels:=DefaultLogLevels;
  for i:=1 to ParamCount do
    begin
      if (ParamStr(i)='-d') or (ParamStr(i)='--debug') then
        begin
          LogLevels:=AllLogLevels+[llDebug];
          break;
        end;
      if (ParamStr(i)='-v') or (ParamStr(i)='--verbose') then
        begin
          LogLevels:=AllLogLevels+[llDebug];
          break;
        end;
    end;
  // First try config file from command line
  if HasOption('C','config-file') then
    cfgfile:=GetOptionValue('C','config-file')
  else
    cfgfile:='';
  GFPpkg.InitializeGlobalOptions(CfgFile);
end;


procedure TMakeTool.MaybeCreateLocalDirs;
begin
  ForceDirectories(GFPpkg.Options.GlobalSection.BuildDir);
  ForceDirectories(GFPpkg.Options.GlobalSection.ArchivesDir);
  ForceDirectories(GFPpkg.Options.GlobalSection.CompilerConfigDir);
end;


procedure TMakeTool.ShowUsage(const aErrorMsg : String = '');

begin
  if (aErrorMsg<>'') then
    Writeln(stdErr,'Error: ',aErrorMsg);
  Writeln('Usage: ',Paramstr(0),' [options] <action> <package>');
  Writeln('Options:');
  Writeln('  -C --config-file   Specify the configuration file to use');
  Writeln('  -c --config        Set compiler configuration to use');
  Writeln('  -g --global        Prefer global configuration file over local configuration file.');
  Writeln('  -h --help          This help');
  Writeln('  -V --version       Show version and exit');
  Writeln('  -v --verbose       Show more information');
  Writeln('  -d --debug         Show debugging information');
  Writeln('  -f --force         Force installation also if the package is already installed');
  Writeln('  -r --recovery      Recovery mode, use always internal fpmkunit');
  Writeln('  -b --broken        Do not stop on broken packages');
  Writeln('  -l --showlocation  Show in which repository the the packages are installed');
  Writeln('  -o --options=value Pass extra options to the compiler');
  Writeln('  -n                 Do not read the default configuration files');
  Writeln('  -p --prefix=value  Specify the prefix');
  Writeln('  -s --skipbroken    Skip the rebuild of depending packages after installation');
  Writeln('  -i --installlocation Specify the repository to install packages into');
  Writeln('  --compiler=value   Specify the compiler-executable');
  Writeln('  --cpu=value        Specify the target cpu to compile for');
  Writeln('  --os=value         Specify the target operating system to compile for');
  Writeln('Actions:');
  Writeln('  update            Update packages list');
  Writeln('  list              List available and installed packages');
  Writeln('  build             Build package');
  Writeln('  compile           Compile package');
  Writeln('  install           Install package');
  Writeln('  uninstall         Uninstall package');
  Writeln('  clean             Clean package');
  Writeln('  archive           Create archive of package');
  Writeln('  download          Download package');
  Writeln('  convertmk         Convert Makefile.fpc to fpmake.pp');
  Writeln('  info              Show more information about a package');
  Writeln('  fixbroken         Recompile all (broken) packages with changed dependencies');
  Writeln('  listsettings      Show the values for all fppkg settings');
  Writeln('  config            Get/Set configuration file values');
//  Writeln('  addconfig          Add a compiler configuration for the supplied compiler');
  Writeln('Config commands:');
  Writeln(' config get a.b            Get setting from config file, section a, key b');
  Writeln(' config get a b            Get setting from config file, section a, key b');
  Writeln(' config set a.b c          Set setting from config file, section a, key b to value c');
  Writeln(' config set a b c          Set setting from config file, section a, key b to value c');
  Halt(Ord(aErrorMsg<>''));
end;

constructor TMakeTool.Create;
begin
  Create(nil);
end;

constructor TMakeTool.Create(aOwner: TComponent);
begin
  Inherited;
  ParaPackages:=TStringList.Create;
end;


destructor TMakeTool.Destroy;
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
      if CheckOption(I,'C','config-file') then
        begin
          // Do nothing, the config-file has already been read.
          OptionArg(I);
        end
      else if CheckOption(I,'c','config') then
        GFPpkg.Options.CommandLineSection.CompilerConfig:=OptionArg(I)
      else if CheckOption(I,'v','verbose') then
        LogLevels:=AllLogLevels
      else if CheckOption(I,'d','debug') then
        LogLevels:=AllLogLevels+[llDebug]
      else if CheckOption(I,'i','installrepository') then
        GFPpkg.Options.CommandLineSection.InstallRepository:=OptionArg(I)
      else if CheckOption(I,'r','recovery') then
        GFPpkg.Options.CommandLineSection.RecoveryMode:=true
      else if CheckOption(I,'n','') then
        GFPpkg.Options.CommandLineSection.SkipConfigurationFiles:=true
      else if CheckOption(I,'b','broken') then
        GFPpkg.Options.CommandLineSection.AllowBroken:=true
      else if CheckOption(I,'l','showlocation') then
        GFPpkg.Options.CommandLineSection.ShowLocation:=true
      else if CheckOption(I,'s','skipbroken') then
        GFPpkg.Options.CommandLineSection.SkipFixBrokenAfterInstall:=true
      else if CheckOption(I,'g','global') then
        GFPpkg.Options.PreferGlobal:=true
      else if CheckOption(I,'o','options') and FirstPass then
        begin
          OptString := OptionArg(I);
          while OptString <> '' do
            GFPpkg.CompilerOptions.Options.Add(SplitSpaces(OptString));
        end
      else if CheckOption(I,'p','prefix') then
        begin
          GFPpkg.CompilerOptions.GlobalPrefix := OptionArg(I);
          GFPpkg.CompilerOptions.LocalPrefix := OptionArg(I);
          GFPpkg.FPMakeCompilerOptions.GlobalPrefix := OptionArg(I);
          GFPpkg.FPMakeCompilerOptions.LocalPrefix := OptionArg(I);
        end
      else if CheckOption(I,'','compiler') then
        begin
          GFPpkg.CompilerOptions.Compiler := OptionArg(I);
          GFPpkg.FPMakeCompilerOptions.Compiler := OptionArg(I);
        end
      else if CheckOption(I,'','os') then
        GFPpkg.CompilerOptions.CompilerOS := StringToOS(OptionArg(I))
      else if CheckOption(I,'','cpu') then
        GFPpkg.CompilerOptions.CompilerCPU := StringToCPU(OptionArg(I))
      else if CheckOption(I,'h','help') then
        begin
          ShowUsage;
          halt(0);
        end
      else if CheckOption(I,'V','version') then
        begin
          ShowVersion;
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
    ShowUsage('No action specified!');
end;

procedure TMakeTool.HandleConfig;

Type
  TConfigMode =  (cfUnknown,cfGet,cfSet);

Const
  cCount : array[TConfigMode] of byte = (0,2,3);

var
  aMode : TConfigMode;
  aIni : TMemIniFile;
  cfgFile,aSection,aKey,aValue : String;

  function GetSectionKey(getValue : boolean) : Boolean;

  var
    p,pValue : Integer;

  begin
    aSection:=ParaPackages[1];
    pValue:=2;
    P:=Pos('.',aSection);
    if P>0 then
      begin
      aKey:=Copy(aSection,P+1,Length(aSection));
      Delete(aSection,P,Length(aSection));
      end
    else
      begin
      if ParaPackages.Count>=3 then
        begin
        aKey:=ParaPackages[2];
        Inc(pValue);
        end;
      end;
    Result:=Not ((aSection='') or (aKey=''));
    if not Result then
      ShowUsage('Config: No section and key specified!')
    else if GetValue then
      if pValue<ParaPackages.Count then
        aValue:=ParaPackages[Pvalue]
      else
        ShowUsage('Config: No value specified!');
    Writeln('S: ',aSection,', K: ',aKey,', V: ',aValue);
  end;

begin
// We know there is at least 1 parapackage
  aIni:=Nil;
  aMode:=cfUnknown;
  //writeln('args: ',parapackages.text);
  if ParaPackages[0]='get' then
    aMode:=cfGet
  else if ParaPackages[0]='set' then
    aMode:=cfSet;
  if aMode=cfUnknown then
    ShowUsage('Config: Unknown config command : '+ParaPackages[0])
  else if Not (ParaPackages.Count in [cCount[aMode],cCount[aMode]+1]) then
    begin
    ShowUsage(Format('Config: Wrong amount of arguments. Expected %d, got %d.',[cCount[aMode],ParaPackages.Count]));
    aMode:=cfUnknown;
    end;
  if HasOption('C','config-file') then
    cfgfile:=GetOptionValue('C','config-file')
  else
    begin
    cfgfile:=GetFppkgConfigFile(GFPpkg.Options.PreferGlobal,false);
    if Not FileExists(cfgFile) then
      cfgfile:=GetFppkgConfigFile(GFPpkg.Options.PreferGlobal,false);
    end;
  Writeln('Getting from file : ',CfgFile);
  if aMode<>cfUnknown then
    aIni:=TMemIniFile.Create(cfgFile);
  Case aMode of
    cfGet :
      begin
      if GetSectionKey(False) then
        aValue:=aIni.ReadString(aSection,aKey,'')
      else
         exit;
      writeln(aValue);
      end;
    cfSet :
      begin
      if GetSectionKey(True) then
        begin
        aIni.WriteString(aSection,aKey,aValue);
        try
          aIni.UpdateFile;
        except
          On EIO: EInoutError do
            Writeln(stderr,'Failed to update file: ',cfgfile,'. Make sure you have sufficient rights to write this file.');
        end;
        end
      else
         exit;
      end;
  else
    ShowUsage;
  end;
end;

procedure TMakeTool.DoRun;
var
  OldCurrDir : String;
  i      : Integer;
  SL     : TStringList;
  Repo: TFPRepository;
  InstPackages: TFPCurrentDirectoryPackagesStructure;
  ArchivePackages: TFPArchiveFilenamePackagesStructure;
begin
  Terminate; // We run only once

  OldCurrDir:=GetCurrentDir;
  Try
    InitializeFppkg;
    LoadGlobalDefaults;
    ProcessCommandLine(true);

    SetLength(FPMKUnitDeps,FPMKUnitDepDefaultCount);
    for i := 0 to FPMKUnitDepDefaultCount-1 do
      FPMKUnitDeps[i]:=FPMKUnitDepsDefaults[i];

    MaybeCreateLocalDirs;
    if not GFPpkg.Options.CommandLineSection.SkipConfigurationFiles then
      begin
        GFPpkg.InitializeCompilerOptions;
        if GFPpkg.Options.GlobalSection.ConfigVersion = 4 then
          begin
            // This version did not have any repository configured, but used a
            // 'local' and 'global' compiler-setting.
            GFPpkg.Options.AddRepositoriesForCompilerSettings(GFPpkg.CompilerOptions);
          end;
      end
    else
      begin
        GFPpkg.FPMakeCompilerOptions.InitCompilerDefaults;
        GFPpkg.CompilerOptions.InitCompilerDefaults;
      end;

    // The command-line is parsed for the second time, to make it possible
    // to override the values in the compiler-configuration file. (like prefix)
    ProcessCommandLine(false);

    // Config command does not do anything except get/set values
    if (ParaAction = 'config') then
      begin
      If ParaPackages.Count=0 then
        ShowUsage('config command needs arguments')
      else
        HandleConfig;
      exit;
      end;


    // If CompilerVersion, CompilerOS or CompilerCPU is still empty, use the
    // compiler-executable to get them
    GFPpkg.FPMakeCompilerOptions.CheckCompilerValues;
    GFPpkg.CompilerOptions.CheckCompilerValues;

    LoadLocalAvailableMirrors;

    // Load local repository, update first if this is a new installation
    // errors will only be reported as warning. The user can be bootstrapping
    // and do an update later
    if not FileExists(GFPpkg.Options.GlobalSection.LocalPackagesFile) then
      begin
        try
          pkghandler.ExecuteAction('','update', GFPpkg);
        except
          on E: Exception do
            pkgglobals.Log(llWarning,E.Message);
        end;
      end;
    FindInstalledPackages(GFPpkg.FPMakeCompilerOptions,true);

    // Check for broken dependencies
    if not GFPpkg.Options.CommandLineSection.AllowBroken and
       (((ParaAction='fixbroken') and (ParaPackages.Count>0)) or
        (ParaAction='compile') or
        (ParaAction='build') or
        (ParaAction='install') or
        (ParaAction='archive')) then
      begin
        pkgglobals.Log(llDebug,SLogCheckBrokenDependenvies);
        SL:=TStringList.Create;
        if FindBrokenPackages(SL) then
          Error(SErrBrokenPackagesFound);
        FreeAndNil(SL);
      end;

    if (ParaAction='install') or (ParaAction='uninstall') or
      (ParaAction='fixbroken') then
      GFPpkg.ScanInstalledPackagesForAvailablePackages;

    if ParaPackages.Count=0 then
      begin
        // Do not add the fake-repository with the contents of the current directory
        // when a list of packages is shown. (The fake repository should not be shown)
        if ParaAction<>'list' then
          begin
            Repo := TFPRepository.Create(GFPpkg);
            GFPpkg.RepositoryList.Add(Repo);
            Repo.RepositoryType := fprtAvailable;
            Repo.RepositoryName := 'CurrentDirectory';
            Repo.Description := 'Package in current directory';
            InstPackages := TFPCurrentDirectoryPackagesStructure.Create(GFPpkg);
            InstPackages.InitializeWithOptions(nil, GFPpkg.Options, GFPpkg.CompilerOptions);
            InstPackages.Path := OldCurrDir;
            InstPackages.AddPackagesToRepository(Repo);
            Repo.DefaultPackagesStructure := InstPackages;
          end;
        pkghandler.ExecuteAction(CurrentDirPackageName,ParaAction,GFPpkg);
      end
    else
      begin
        // Process packages
        for i:=0 to ParaPackages.Count-1 do
          begin
            if sametext(ExtractFileExt(ParaPackages[i]),'.zip') and FileExists(ParaPackages[i]) then
              begin
                Repo := TFPRepository.Create(GFPpkg);
                GFPpkg.RepositoryList.Add(Repo);
                Repo.RepositoryType := fprtAvailable;
                Repo.RepositoryName := 'ArchiveFile';
                Repo.Description := 'Package in archive-file';
                ArchivePackages := TFPArchiveFilenamePackagesStructure.Create(GFPpkg);
                ArchivePackages.InitializeWithOptions(nil, GFPpkg.Options, GFPpkg.CompilerOptions);
                ArchivePackages.ArchiveFileName := ParaPackages[i];
                ArchivePackages.AddPackagesToRepository(Repo);
                Repo.DefaultPackagesStructure := ArchivePackages;

                pkgglobals.Log(llDebug,SLogCommandLineAction,['['+CmdLinePackageName+']',ParaAction]);
                pkghandler.ExecuteAction(CmdLinePackageName,ParaAction,GFPpkg);
              end
            else
              begin
                pkgglobals.Log(llDebug,SLogCommandLineAction,['['+ParaPackages[i]+']',ParaAction]);
                pkghandler.ExecuteAction(ParaPackages[i],ParaAction,GFPpkg);
              end;
          end;
      end;

    // Recompile all packages dependent on this package
    if (ParaAction='install') and not GFPpkg.Options.CommandLineSection.SkipFixBrokenAfterInstall then
      pkghandler.ExecuteAction('','fixbroken',GFPpkg);


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

procedure TMakeTool.ShowVersion;
var
  Version: TFPVersion;
begin
  Version := TFPVersion.Create;
  try
    Version.Major := version_major;
    Version.Minor := version_minor;
    Version.Micro := version_micro;
    Version.Build := version_build;
    Writeln('Version: ', Version.AsString);
  finally
    Version.Free;
  end;
end;


begin
  With TMakeTool.Create do
    try
      run;
    finally
      Free;
    end;
end.

