unit pkgfpmake;

{$mode objfpc}{$H+}

interface

uses
  Classes,SysUtils,DateUtils,
  pkghandler, fpmkunit;

implementation

uses
  fprepos,
  pkgoptions,
  pkgglobals,
  pkgmessages,
  pkgrepos,
  pkgFppkg,
  fpxmlrep;

type
  { TFPMakeCompiler }

  TFPMakeCompiler = Class(TPackagehandler)
  protected
    function DeterminePackage: TFPPackage;
  Public
    function Execute: Boolean; override;
  end;


  { TFPMakeRunner }

  TFPMakeRunner = Class(TFPMakeCompiler)
  Protected
    function IncludeInstallationOptions: Boolean; virtual;
    Function RunFPMake(const Command:string):Integer;
  end;


  { TFPMakeRunnerCompile }

  TFPMakeRunnerCompile = Class(TFPMakeRunner)
  Public
    function Execute: Boolean;override;
  end;


  { TFPMakeRunnerBuild }

  TFPMakeRunnerBuild = Class(TFPMakeRunner)
  Public
    function Execute: Boolean;override;
  end;


  { TFPMakeRunnerInstall }

  TFPMakeRunnerInstall = Class(TFPMakeRunner)
  protected
    function IncludeInstallationOptions: Boolean; override;
  Public
    function Execute: Boolean;override;
  end;

  { TFPMakeRunnerUnInstall }

  TFPMakeRunnerUnInstall = Class(TFPMakeRunner)
  protected
    function IncludeInstallationOptions: Boolean; override;
  Public
    function Execute: Boolean;override;
  end;

  { TFPMakeRunnerClean }

  TFPMakeRunnerClean = Class(TFPMakeRunner)
  Public
    function Execute: Boolean;override;
  end;

  { TFPMakeRunnerManifest }

  TFPMakeRunnerManifest = Class(TFPMakeRunner)
  Public
    function Execute: Boolean;override;
  end;

  { TFPMakeRunnerArchive }

  TFPMakeRunnerArchive = Class(TFPMakeRunner)
  Public
    function Execute: Boolean;override;
  end;

   TMyMemoryStream=class(TMemoryStream)
   public
     constructor Create(p:pointer;mysize:integer);
   end;

{
  Generated from fpmkunit.pp, using data2inc:
  data2inc -b -s fpmkunit.pp fpmkunitsrc.inc fpmkunitsrc
}
{$i fpmkunitsrc.inc}

procedure CreateFPMKUnitSource(const AFileName:string);
var
  InStream,
  OutStream : TStream;
  pend      : pchar;
begin
  try
    // Don't write trailing #0
    pend:=pchar(@fpmkunitsrc)+sizeof(fpmkunitsrc)-1;
    while pend^=#0 do
      dec(pend);
    InStream:=TMyMemoryStream.Create(@fpmkunitsrc,pend-pchar(@fpmkunitsrc));
    OutStream:=TFileStream.Create(AFileName,fmCreate);
    OutStream.CopyFrom(InStream,InStream.Size);
  finally
    InStream.Destroy;
    OutStream.Destroy;
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


{ TFPMakeCompiler }

function TFPMakeCompiler.DeterminePackage: TFPPackage;
begin
  Result := PackageManager.DetermineSourcePackage(PackageName);

  if not Assigned(Result) then
    Raise EPackage.CreateFmt(SErrMissingPackage,[PackageName]);
end;

function TFPMakeCompiler.Execute: Boolean;
var
  OOptions : string;

  function CheckUnitDir(const APackageName:string;Out AUnitDir:string):boolean;
  var
    P: TFPPackage;
  begin
    Result:=false;
    P := PackageManager.FPMakeRepoFindPackage(APackageName, pkgpkInstalled);
    if Assigned(P) then
      begin
        AUnitDir := P.PackagesStructure.GetUnitDirectory(P);
        Result := DirectoryExistsLog(AUnitDir);
      end;
  end;

  procedure AddOption(const s:string);
  begin
    if OOptions<>'' then
      OOptions:=OOptions+' ';
    OOptions:=OOptions+maybequoted(s);
  end;

  procedure AddDependencySearchPaths(ADepDirList: TStrings; APackageName: string);
  var
    i: Integer;
    D: TFPDependency;
    UnitDir: string;
    Package: TFPPackage;
  begin
    Package := PackageManager.FindPackage(APackageName, pkgpkInstalled);
    if not assigned(Package) then
      begin
        Error(SErrMissingInstallPackage, [PackageName]);
      end;
    for i := 0 to Package.Dependencies.Count -1 do
      begin
        D := Package.Dependencies[i];
        if (PackageManager.CompilerOptions.CompilerOS in D.OSes) and
           (PackageManager.CompilerOptions.CompilerCPU in D.CPUs) then
          begin
            if CheckUnitDir(D.PackageName, UnitDir) then
              begin
                AddDependencySearchPaths(ADepDirList, D.PackageName);
                ADepDirList.Add(UnitDir);
              end
            else
              Error(SErrMissingInstallPackage, [D.PackageName]);
          end;
      end;
  end;

Var
  i : Integer;
  TempBuildDir,
  DepDir,
  FPMakeBin,
  FPMakeSrc : string;
  NeedFPMKUnitSource,
  HaveFpmake : boolean;
  FPMKUnitDepAvailable: Boolean;
  FPMKUnitDepPackage: TFPPackage;
  P : TFPPackage;
  DepPackage: TFPPackage;
  DepDirList: TStringList;
  S,Reason: string;
begin
  Result := True;
  P:=DeterminePackage;
  NeedFPMKUnitSource:=false;
  OOptions:='';
  SetCurrentDir(PackageManager.PackageBuildPath(P));
  // Generate random name for build path
  TempBuildDir:='build_fpmake_'+HexStr(DateTimeToUnix(Now),8)+HexStr(GetProcessId,4);
  // Check for fpmake source
  FPMakeBin:='fpmake'+ExeExt;
  FPMakeSrc:='fpmake.pp';
  HaveFpmake:=FileExists(FPMakeSrc);
  If Not HaveFPMake then
    begin
      HaveFPMake:=FileExists('fpmake.pas');
      If HaveFPMake then
        FPMakeSrc:='fpmake.pas';
    end;
  // Need to compile fpmake executable?
  if not FileExists(FPMakeBin) or
     (FileAge(FPMakeBin)<FileAge(FPMakeSrc)) then
    begin
      if Not HaveFPMake then
        Error(SErrMissingFPMake);
      AddOption('-n');
      AddOption('-dCOMPILED_BY_FPPKG');

      DepDirList := TStringList.Create;
      try
        DepDirList.Sorted := True;
        DepDirList.Duplicates := dupIgnore;
        for i:=0 to high(FPMKUnitDeps) do
          begin
            FPMKUnitDepAvailable := FPMKUnitDeps[i].available;
            if FPMKUnitDepAvailable then
              begin
                // Do not try to use packages which are broken. This can happen when
                // fixbroken is being called and one of the fpmkunit-dependencies itself
                // is broken. In that case, build fpmake without the dependency.
                // (Plugins are also fpmkunit-dependencies)
                FPMKUnitDepPackage := PackageManager.FindPackage(FPMKUnitDeps[i].package, pkgpkInstalled);
                if Assigned(FPMKUnitDepPackage) then
                  begin
                    if PackageManager.PackageIsBroken(FPMKUnitDepPackage, Reason, nil) then
                      FPMKUnitDepAvailable := False;
                  end;
              end;

            if FPMKUnitDepAvailable then
              begin
                if CheckUnitDir(FPMKUnitDeps[i].package,DepDir) then
                  begin
                    AddDependencySearchPaths(DepDirList, FPMKUnitDeps[i].package);
                    DepDirList.Add(DepDir);
                  end
                else
                  Error(SErrMissingInstallPackage,[FPMKUnitDeps[i].package]);
                if FPMKUnitDeps[i].def<>'' then
                  AddOption('-d'+FPMKUnitDeps[i].def);
                if FPMKUnitDeps[i].PluginUnit<>'' then
                  AddOption('-Fa'+FPMKUnitDeps[i].PluginUnit);
              end
            else
              begin
                // If fpmkunit is not installed, we use the internal fpmkunit source
                if FPMKUnitDeps[i].package='fpmkunit' then
                  begin
                    NeedFPMKUnitSource:=true;
                    DepDirList.Add(TempBuildDir);
                  end;
                if FPMKUnitDeps[i].undef<>'' then
                  AddOption('-d'+FPMKUnitDeps[i].undef);
              end;
          end;
        // Add RTL unit dir
        if not CheckUnitDir('rtl',DepDir) then
          Error(SErrMissingInstallPackage,['rtl']);
        DepDirList.Add(DepDir);
        DepDirList.Add(TempBuildDir);
        for i := 0 to DepDirList.Count -1 do
          AddOption('-Fu'+DepDirList[i]);
      finally
        DepDirList.Free;
      end;
      // Units in a directory for easy cleaning
      DeleteDir(TempBuildDir);
      ForceDirectories(TempBuildDir);
      try
        // Compile options
        //   -- default is to optimize, smartlink and strip to reduce
        //      the executable size (there can be 100's of fpmake's on a system)
        if llInfo in LogLevels then
          AddOption('-vi');
        AddOption('-O2');
        AddOption('-XXs');
        if PackageManager.FPMakeCompilerOptions.HasOptions then
          for S in PackageManager.FPMakeCompilerOptions.Options do
            AddOption(S);
         // AddOption(PackageManager.FPMakeCompilerOptions.Options.DelimitedText) ;
        // Create fpmkunit.pp if needed
        if NeedFPMKUnitSource then
          begin
            Log(llWarning,SLogUseInternalFpmkunit);
            CreateFPMKUnitSource(TempBuildDir+PathDelim+'fpmkunit.pp');
          end;
        // Call compiler
        If ExecuteProcess(PackageManager.FPMakeCompilerOptions.Compiler,OOptions+' '+FPmakeSrc)<>0 then
          begin
            if not PackageManager.Options.CommandLineSection.RecoveryMode then
              Error(SErrCompileFailureFPMakeTryRecovery)
            else
              Error(SErrCompileFailureFPMake);
          end;
      finally
        // Cleanup units
        DeleteDir(TempBuildDir);
      end;
    end
  else
    Log(llCommands,SLogNotCompilingFPMake);
end;


{ TFPMakeRunner }

function TFPMakeRunner.IncludeInstallationOptions: Boolean;
begin
  Result := False;
end;

Function TFPMakeRunner.RunFPMake(const Command:string) : Integer;
Var
  P : TFPPackage;
  FPMakeBin,
  OOptions : string;
  InstallRepo: TFPRepository;
  GlobalUnitDir,
  LocalUnitDir: string;
  BaseInstDir: string;
  i: Integer;

  procedure AddOption(const s:string);
  begin
    if OOptions<>'' then
      OOptions:=OOptions+' ';
    OOptions:=OOptions+maybequoted(s);
    Writeln('Options: >>>>',OOptions,'<<<<');
  end;

  procedure CondAddOption(const Name,Value:string);
  begin
    if Value<>'' then
      AddOption(Name+'='+Value);
  end;

  procedure ObtainSupportedTargetsFromManifest(p:TFPPackage);
  var
    X : TFPXMLRepositoryHandler;
    ManifestPackages : TFPPackages;
    i: integer;
  begin
    p.OSes:=[];
    p.CPUs:=[];
    ManifestPackages:=TFPPackages.Create(TFPPackage);
    X:=TFPXMLRepositoryHandler.Create;
    try
      X.LoadFromXml(ManifestPackages,ManifestFileName);
      for i := 0 to ManifestPackages.Count-1 do
        begin
          p.OSes:=p.OSes+ManifestPackages[i].OSes;
          p.CPUs:=p.CPUs+ManifestPackages[i].CPUs;
        end;
    finally
      X.Free;
      ManifestPackages.Free;
    end;
  end;

begin
  Result := -1;
  OOptions:='';
  // Does the current package support this CPU-OS?
  if PackageName<>'' then
    begin
      P := DeterminePackage;
      if (PackageName=CurrentDirPackageName) and (FileExists(ManifestFileName)) then
        ObtainSupportedTargetsFromManifest(p);
    end
  else
    P:=nil;
  if assigned(P) then
    begin
      if (command<>'archive') and (command<>'manifest') and
         (not(PackageManager.CompilerOptions.CompilerOS in P.OSes) or
          not(PackageManager.CompilerOptions.CompilerCPU in P.CPUs)) then
        begin
          Error(SErrPackageDoesNotSupportTarget,[P.Name,MakeTargetString(PackageManager.CompilerOptions.CompilerCPU,PackageManager.CompilerOptions.CompilerOS)]);
          Exit;
        end;
    end;
  { Maybe compile fpmake executable? }
  if not ExecuteAction(PackageName,'compilefpmake') then
    Exit;
  { Create options }
  if llDebug in LogLevels then
    AddOption('--debug')
  else if llInfo in LogLevels then
    AddOption('--verbose');
  if (P.FPMakeOptionsString<>'') then
    begin
      // When the package is being reinstalled because of broken dependencies, use the same fpmake-options
      // as were used to compile the package in the first place.
      OOptions:=P.FPMakeOptionsString;
    end
  else
    begin
      if PackageManager.CompilerOptions.HasOptions then
        AddOption('--options='+PackageManager.CompilerOptions.Options.DelimitedText);

      if PackageManager.Options.GlobalSection.CustomFPMakeOptions<>'' then
        begin
        AddOption('--ignoreinvalidoption');
        AddOption(PackageManager.Options.GlobalSection.CustomFPMakeOptions);
        end;
    end;

  AddOption('--nofpccfg');
  AddOption('--compiler='+PackageManager.CompilerOptions.Compiler);
  AddOption('--cpu='+CPUToString(PackageManager.CompilerOptions.CompilerCPU));
  AddOption('--os='+OSToString(PackageManager.CompilerOptions.CompilerOS));

  // While scanning a source-repository it could be necessary to create manifest
  // files. At this moment the InstallRepo could not be initialized yet. And the
  // manifest command does not use the --prefix and --baseinstalldir parameters.
  if IncludeInstallationOptions then
    begin
      InstallRepo := PackageManager.GetInstallRepository(P);

      if not Assigned(InstallRepo.DefaultPackagesStructure) then
        begin
          Error(SErrIllConfRepository,[InstallRepo.RepositoryName]);
          Exit;
        end;
      CondAddOption('--baseinstalldir',InstallRepo.DefaultPackagesStructure.GetBaseInstallDir);
      CondAddOption('--prefix',InstallRepo.DefaultPackagesStructure.GetPrefix);
    end;

  for i := PackageManager.RepositoryList.Count-1 downto 0 do
    begin
      if PackageManager.RepositoryList[i] is TFPRepository then
        begin
          InstallRepo := TFPRepository(PackageManager.RepositoryList[i]);
          if (InstallRepo.RepositoryType = fprtInstalled) and Assigned(InstallRepo.DefaultPackagesStructure) then
            begin
              BaseInstDir := InstallRepo.DefaultPackagesStructure.GetBaseInstallDir;
              CondAddOption('--searchpath', BaseInstDir);
              if LocalUnitDir='' then
                LocalUnitDir := BaseInstDir
              else if GlobalUnitDir='' then
                GlobalUnitDir := BaseInstDir;
            end;
        end;
    end;
  if GlobalUnitDir<>'' then
    CondAddOption('--globalunitdir', GlobalUnitDir);
  if LocalUnitDir<>'' then
    CondAddOption('--localunitdir', LocalUnitDir);

  { Run FPMake }
  FPMakeBin:='fpmake'+ExeExt;
  SetCurrentDir(PackageManager.PackageBuildPath(P));
  Result:=ExecuteProcess(FPMakeBin,Command+' '+OOptions);
  if Result<>0 then
    Error(SErrExecutionFPMake,[Command]);
end;


function TFPMakeRunnerCompile.Execute: Boolean;
begin
  Result := RunFPMake('compile') = 0;
end;


function TFPMakeRunnerBuild.Execute: Boolean;
begin
  Result := RunFPMake('build') = 0;
end;

function TFPMakeRunnerInstall.IncludeInstallationOptions: Boolean;
begin
  Result := True;
end;

function TFPMakeRunnerInstall.Execute: Boolean;
begin
  Result := RunFPMake('install') = 0;
end;

function TFPMakeRunnerUnInstall.IncludeInstallationOptions: Boolean;
begin
  Result := True;
end;

function TFPMakeRunnerUnInstall.Execute: Boolean;
begin
  Result := RunFPMake('uninstall') = 0;
end;


function TFPMakeRunnerClean.Execute: Boolean;
begin
  Result := RunFPMake('clean') = 0;
end;


function TFPMakeRunnerManifest.Execute: Boolean;
begin
  Result := RunFPMake('manifest') = 0;
end;


function TFPMakeRunnerArchive.Execute: Boolean;
var
  StoredLocalPrefix: string;
  StoredGlobalPrefix: string;
begin
  // In most (all?) cases we do not want a prefix in the archive.
  StoredGlobalPrefix := PackageManager.CompilerOptions.GlobalPrefix;
  StoredLocalPrefix := PackageManager.CompilerOptions.LocalPrefix;
  PackageManager.CompilerOptions.GlobalPrefix := '';
  PackageManager.CompilerOptions.LocalPrefix := '';
  try
    Result := RunFPMake('archive') = 0;
  finally
    PackageManager.CompilerOptions.GlobalPrefix := StoredGlobalPrefix;
    PackageManager.CompilerOptions.LocalPrefix := StoredLocalPrefix;
  end;
end;


initialization
  RegisterPkgHandler('compilefpmake',TFPMakeCompiler);
  RegisterPkgHandler('fpmakecompile',TFPMakeRunnerCompile);
  RegisterPkgHandler('fpmakebuild',TFPMakeRunnerBuild);
  RegisterPkgHandler('fpmakeinstall',TFPMakeRunnerInstall);
  RegisterPkgHandler('fpmakeuninstall',TFPMakeRunnerUnInstall);
  RegisterPkgHandler('fpmakeclean',TFPMakeRunnerClean);
  RegisterPkgHandler('fpmakemanifest',TFPMakeRunnerManifest);
  RegisterPkgHandler('fpmakearchive',TFPMakeRunnerArchive);
end.
