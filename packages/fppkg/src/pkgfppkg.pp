unit pkgFppkg;

{$mode objfpc}{$H+}

interface

uses
  Classes,
  SysUtils,
  contnrs,
  fprepos,
  pkgmessages,
  pkgglobals,
  pkgoptions,
  pkgPackagesStructure;

type

  { TpkgFPpkg }

  TpkgPackageKind = (pkgpkInstalled, pkgpkAvailable, pkgpkBoth);
  TpkgFPpkg = class(TComponent)
  private
    FInsideFindBrokenPackages: Integer;
    FBrokenPackagesDictionary: TFPHashList;

    FFPMakeRepositoryList: TComponentList;
    FRepositoryList: TComponentList;
    FOptions: TFppkgOptions;
    FCompilerOptions: TCompilerOptions;
    FFpmakeCompilerOptions: TCompilerOptions;
    FCurrentRemoteRepositoryURL: String;
    FConfigurationFilename: string;
    function IncludeRepositoryTypeForPackageKind(ARepositoryType: TFPRepositoryType;
      APackageKind: TpkgPackageKind): Boolean;
    procedure ScanPackagesOnDisk(ACompilerOptions: TCompilerOptions; APackageKind: TpkgPackageKind; ARepositoryList: TComponentList);
    function CreateRepository(ARepoOptionSection: TFppkgRepositoryOptionSection;
      AnOptions: TFppkgOptions; ACompilerOptions: TCompilerOptions): TFPRepository;
    function  FindPackage(ARepositoryList: TComponentList; APackageName: string; APackageKind: TpkgPackageKind): TFPPackage;

    function  SelectRemoteMirror:string;
    procedure EnterFindBrokenPackages;
    procedure LeaveFindBrokenpackages;

    procedure ClearRepositories(ARepositoryList: TComponentList);
    function GetConfigurationFilename: string;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;

    procedure InitializeGlobalOptions(CfgFile: string);
    procedure InitializeCompilerOptions;
    procedure LoadLocalAvailableMirrors;
    procedure ScanAvailablePackages;
    procedure ScanPackages;

    function PackageIsBroken(APackage: TFPPackage; out Reason: string; ARepository: TFPRepository): Boolean;

    function FPMakeRepoFindPackage(APackageName: string; APackageKind: TpkgPackageKind): TFPPackage;
    function FindPackage(APackageName: string; APackageKind: TpkgPackageKind): TFPPackage;
    function PackageByName(APackageName: string; APackageKind: TpkgPackageKind): TFPPackage;

    function FindRepository(ARepositoryName: string): TFPRepository;
    function RepositoryByName(ARepositoryName: string): TFPRepository;

    function GetInstallRepository(ASourcePackage: TFPPackage): TFPRepository;
    function DetermineSourcePackage(APackageName: String): TFPPackage;
    function PackageLocalArchive(APackage:TFPPackage): String;
    function PackageBuildPath(APackage:TFPPackage):String;

    function GetRemoteRepositoryURL(const AFileName:string):string;
    function PackageRemoteArchive(APackage:TFPPackage): String;

    procedure ScanInstalledPackagesForAvailablePackages;
    procedure CheckFPMakeDependencies;
    function  FindBrokenPackages(SL:TStrings):Boolean;

    property Options: TFppkgOptions read FOptions;
    property CompilerOptions: TCompilerOptions read FCompilerOptions;
    property FpmakeCompilerOptions: TCompilerOptions read FFpmakeCompilerOptions;
    property FPMakeRepositoryList: TComponentList read FFPMakeRepositoryList;
    property RepositoryList: TComponentList read FRepositoryList;
    property ConfigurationFilename: string read GetConfigurationFilename;
  public

  end;

implementation

uses
  fpmkunit,
  fpxmlrep,
  pkgrepos;

{ TpkgFPpkg }

constructor TpkgFPpkg.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FOptions := TFppkgOptions.Create;
  FCompilerOptions := TCompilerOptions.Create;
  FFpmakeCompilerOptions := TCompilerOptions.Create;
  FRepositoryList := TComponentList.Create(False);
  FFPMakeRepositoryList := TComponentList.Create(False);
  FBrokenPackagesDictionary := TFPHashList.Create;
end;

destructor TpkgFPpkg.Destroy;
begin
  FBrokenPackagesDictionary.Free;
  FFPMakeRepositoryList.Free;
  FRepositoryList.Free;
  FCompilerOptions.Free;
  FFpmakeCompilerOptions.Free;
  FOptions.Free;
  inherited Destroy;
end;

function TpkgFPpkg.IncludeRepositoryTypeForPackageKind(ARepositoryType: TFPRepositoryType;
  APackageKind: TpkgPackageKind): Boolean;
begin
  Result := ((APackageKind=pkgpkInstalled) and (ARepositoryType = fprtInstalled)) or
    ((APackageKind=pkgpkAvailable) and (ARepositoryType = fprtAvailable)) or
    (APackageKind=pkgpkBoth);
end;

procedure TpkgFPpkg.ScanPackagesOnDisk(ACompilerOptions: TCompilerOptions;
  APackageKind: TpkgPackageKind; ARepositoryList: TComponentList);
var
  i: Integer;
  RepoOption: TFppkgRepositoryOptionSection;
  Repo: TFPRepository;
begin
  FOptions.BindToCompilerOptions(ACompilerOptions);
  for i := 0 to FOptions.SectionList.Count -1 do
    begin
      if FOptions.SectionList[i] is TFppkgRepositoryOptionSection then
        begin
          RepoOption := TFppkgRepositoryOptionSection(FOptions.SectionList[i]);
          if IncludeRepositoryTypeForPackageKind(RepoOption.GetRepositoryType, APackageKind) then
            begin
              Repo := CreateRepository(RepoOption, FOptions, ACompilerOptions);
              if Assigned(Repo) then
                begin
                  ARepositoryList.Add(Repo);
                  if Assigned(Repo.DefaultPackagesStructure) then
                    Repo.DefaultPackagesStructure.AddPackagesToRepository(Repo);
                end;
            end;
        end;
    end;
end;

function TpkgFPpkg.CreateRepository(ARepoOptionSection: TFppkgRepositoryOptionSection;
  AnOptions: TFppkgOptions; ACompilerOptions: TCompilerOptions): TFPRepository;
begin
  Result := TFPRepository.Create(Self);
  Result.InitializeWithOptions(ARepoOptionSection, AnOptions, ACompilerOptions);
end;

procedure TpkgFPpkg.InitializeGlobalOptions(CfgFile: string);
var
  GeneratedConfig: boolean;
  FirstRepoConf: TFppkgOptionSection;
begin
  GeneratedConfig:=false;
  // First try specified config file
  if (CfgFile<>'') then
    begin
      if not FileExists(cfgfile) then
        Error(SErrNoSuchFile,[cfgfile]);
    end
  else
    begin
      // Now try if a local config-file exists
      cfgfile:=GetFppkgConfigFile(Options.PreferGlobal,False);
      if not FileExists(cfgfile) then
        begin
          // If not, try to find a global configuration file
          cfgfile:=GetFppkgConfigFile(not Options.PreferGlobal,False);
          if not FileExists(cfgfile) then
            begin
              // Create a new configuration file
              if not IsSuperUser then // Make a local, not global, configuration file
                cfgfile:=GetFppkgConfigFile(False,False);
              ForceDirectories(ExtractFilePath(cfgfile));
              FOptions.SaveToFile(cfgfile);
              GeneratedConfig:=true;
            end;
        end;
    end;
  // Load file or create new default configuration
  if not GeneratedConfig then
    begin
      FOptions.LoadFromFile(cfgfile);
    end;
  FOptions.CommandLineSection.CompilerConfig:=FOptions.GlobalSection.CompilerConfig;
  // Tracing of what we've done above, need to be done after the verbosity is set
  if GeneratedConfig then
    pkgglobals.Log(llDebug,SLogGeneratingGlobalConfig,[cfgfile])
  else
    pkgglobals.Log(llDebug,SLogLoadingGlobalConfig,[cfgfile]);
  FConfigurationFilename := CfgFile;
  // Log configuration
  FOptions.LogValues(llDebug);
end;

procedure TpkgFPpkg.InitializeCompilerOptions;
var
  S : String;
begin
  // Load default compiler config
  S:=FOptions.GlobalSection.CompilerConfigDir+FOptions.CommandLineSection.CompilerConfig;
  FCompilerOptions.UpdateLocalRepositoryOption(FOptions);
  if FileExists(S) then
    begin
      pkgglobals.Log(llDebug,SLogLoadingCompilerConfig,[S]);
      FCompilerOptions.LoadCompilerFromFile(S);
      if FCompilerOptions.SaveInifileChanges then
        // The file is in an old format, try to update the file but ignore
        // any failures.
        FCompilerOptions.SaveCompilerToFile(S);
    end
  else
    begin
      if FCompilerOptions.SaveInifileChanges then
        // A new fppkg.cfg has been created, try to create a new compiler-configuration
        // file too.
        begin
          pkgglobals.Log(llDebug,SLogGeneratingCompilerConfig,[S]);
          FCompilerOptions.InitCompilerDefaults;
          if not FCompilerOptions.SaveCompilerToFile(S) then
            Error(SErrMissingCompilerConfig,[S]);
        end
      else
        Error(SErrMissingCompilerConfig,[S]);
    end;
  // Log compiler configuration
  FCompilerOptions.LogValues(llDebug,'');
  // Load FPMake compiler config, this is normally the same config as above
  S:=FOptions.GlobalSection.CompilerConfigDir+FOptions.GlobalSection.FPMakeCompilerConfig;
  FFPMakeCompilerOptions.UpdateLocalRepositoryOption(FOptions);
  if FileExists(S) then
    begin
      pkgglobals.Log(llDebug,SLogLoadingFPMakeCompilerConfig,[S]);
      FFPMakeCompilerOptions.LoadCompilerFromFile(S);
      if FFPMakeCompilerOptions.SaveInifileChanges then
        // The file is in an old format, try to update the file but ignore
        // any failures.
        FFPMakeCompilerOptions.SaveCompilerToFile(S);
    end
  else
    Error(SErrMissingCompilerConfig,[S]);
  // Log compiler configuration
  FFPMakeCompilerOptions.LogValues(llDebug,'fpmake-building');
end;

procedure TpkgFPpkg.LoadLocalAvailableMirrors;
var
  S : String;
  X : TFPXMLMirrorHandler;
begin
  if assigned(AvailableMirrors) then
    AvailableMirrors.Free;
  AvailableMirrors:=TFPMirrors.Create(TFPMirror);

  // Repository
  S:=Options.GlobalSection.LocalMirrorsFile;
  log(llDebug,SLogLoadingMirrorsFile,[S]);
  if not FileExists(S) then
    exit;
  try
    X:=TFPXMLMirrorHandler.Create;
    With X do
      try
        LoadFromXml(AvailableMirrors,S);
      finally
        Free;
      end;
  except
    on E : Exception do
      begin
        Log(llError,E.Message);
        Error(SErrCorruptMirrorsFile,[S]);
      end;
  end;
end;

procedure TpkgFPpkg.ScanAvailablePackages;
var
  Repo: TFPRepository;
  InstPackages: TFPCustomPackagesStructure;
begin
  if (FOptions.GlobalSection.RemoteMirrorsURL<>'') or
    ((FOptions.GlobalSection.RemoteRepository<>'') and (FOptions.GlobalSection.RemoteRepository<>'auto')) then
    begin
      // In case of a re-scan (for example after an update), remove the old list
      Repo := FindRepository('Available');
      if Assigned(Repo) then
        begin
          RepositoryList.Remove(Repo);
          Repo.Free;
        end;

      Repo := TFPRepository.Create(Self);
      FRepositoryList.Add(Repo);
      Repo.RepositoryName := 'Available';
      Repo.Description := 'Packages available for download';
      Repo.RepositoryType := fprtAvailable;
      InstPackages := TFPRemotePackagesStructure.Create(Self);
      InstPackages.InitializeWithOptions(Nil, FOptions, FCompilerOptions);
      InstPackages.AddPackagesToRepository(Repo);
      Repo.DefaultPackagesStructure := InstPackages;
    end;
end;

procedure TpkgFPpkg.ScanPackages;
begin
  // There is no need to scan for available packages and add them to the
  // FPMakeRepositoryList. Beside that it could lead to problems
  // when the scan of one of the available-repositories tries to compile an
  // fpmake-executable. (Like TFPUninstalledSourcesAvailablePackagesStructure does)
  ClearRepositories(FPMakeRepositoryList);
  ScanPackagesOnDisk(FFpmakeCompilerOptions, pkgpkInstalled, FPMakeRepositoryList);

  CheckFPMakeDependencies;

  ClearRepositories(RepositoryList);
  ScanPackagesOnDisk(FCompilerOptions, pkgpkBoth, RepositoryList);
  ScanAvailablePackages;
end;

function TpkgFPpkg.PackageIsBroken(APackage: TFPPackage; out Reason: string; ARepository: TFPRepository): Boolean;
var
  j, i, ThisRepositoryIndex: Integer;
  Dependency: TFPDependency;
  Repository: TFPRepository;
  DepPackage: TFPPackage;
  HashPtr: PtrInt;
begin
  result:=false;
  Reason := '';
  if Assigned(APackage.Repository) and (APackage.Repository.RepositoryType <> fprtInstalled) then
    begin
    Exit;
    end;

  EnterFindBrokenPackages;
  try
    HashPtr := PtrInt(FBrokenPackagesDictionary.Find(APackage.Name));
    if HashPtr<>0 then
      begin
        // Package is already evaluated
        Result := (HashPtr = 1);
        Exit;
      end;
    if not Assigned(ARepository) then
      begin
      // Check with all repositories
      ThisRepositoryIndex := RepositoryList.Count -1;
      end
    else
      begin
      // We should only check for dependencies in this repository, or repositories
      // with a lower priority.

      // This behaviour seems obsolete. The idea behind it was that each repository
      // should be useable, only using other repositories with a lower priority.
      // In practice this does not work, you have to consider the installation
      // as a whole, using all repositories. One specific user might not be able
      // to 'fix' the global fpc-repository, and so end up with broken packages
      // which he/she can not fix. Or packages may be forced to be installed in
      // a specific repository.
      // The functionality is kept for now, maybe there is a need for it in the
      // future... But for now, ARepository will be always nil.
      ThisRepositoryIndex := -1;
      for i := RepositoryList.Count -1 downto 0 do
        begin
          if RepositoryList.Items[i] = ARepository then
            ThisRepositoryIndex := i;
        end;
      end;

    for j:=0 to APackage.Dependencies.Count-1 do
      begin
        Dependency:=APackage.Dependencies[j];
        if (CompilerOptions.CompilerOS in Dependency.OSes) and
           (CompilerOptions.CompilerCPU in Dependency.CPUs) then
          begin
            DepPackage := nil;
            for i := ThisRepositoryIndex downto 0 do
              begin
                Repository := RepositoryList.Items[i] as TFPRepository;
                if Repository.RepositoryType=fprtInstalled then
                  DepPackage := Repository.FindPackage(Dependency.PackageName);
                if Assigned(DepPackage) then
                  Break;
              end;

            if assigned(DepPackage) then
              begin
                if PackageIsBroken(DepPackage, Reason, ARepository) then
                  begin
                    log(llInfo,SLogPackageDepBroken,[APackage.Name,APackage.Repository.RepositoryName,Dependency.PackageName,Repository.RepositoryName]);
                    result:=true;
                    Reason := Format(SInfoPackageDepBroken, [Dependency.PackageName, Repository.RepositoryName]);
                    FBrokenPackagesDictionary.Add(APackage.Name, Pointer(1));
                    exit;
                  end;
                if (Dependency.RequireChecksum<>$ffffffff) and (DepPackage.Checksum<>Dependency.RequireChecksum) then
                  begin
                    log(llInfo,SLogPackageChecksumChanged,[APackage.Name,APackage.Repository.RepositoryName,Dependency.PackageName,Repository.RepositoryName]);
                    result:=true;
                    Reason := Format(SInfoPackageChecksumChanged, [Dependency.PackageName, Repository.RepositoryName]);
                    FBrokenPackagesDictionary.Add(APackage.Name, Pointer(1));
                    exit;
                  end;
              end
            else
              begin
                log(llInfo,SDbgObsoleteDependency,[APackage.Name,Dependency.PackageName]);
                result:=true;
                Reason :=Format(SInfoObsoleteDependency, [Dependency.PackageName]);
                FBrokenPackagesDictionary.Add(APackage.Name, Pointer(1));
                exit;
              end;
          end;
      end;
    FBrokenPackagesDictionary.Add(APackage.Name, Pointer(2));
  finally
    LeaveFindBrokenpackages;
  end;
end;

function TpkgFPpkg.FPMakeRepoFindPackage(APackageName: string;
  APackageKind: TpkgPackageKind): TFPPackage;
begin
  Result := FindPackage(FPMakeRepositoryList, APackageName, APackageKind);
end;

function TpkgFPpkg.FindPackage(APackageName: string;
  APackageKind: TpkgPackageKind): TFPPackage;
begin
  Result := FindPackage(RepositoryList, APackageName, APackageKind);
end;

function TpkgFPpkg.FindPackage(ARepositoryList: TComponentList; APackageName: string; APackageKind: TpkgPackageKind): TFPPackage;
var
  i: Integer;
  Repo: TFPRepository;
begin
  Result := nil;
  for i := ARepositoryList.Count-1 downto 0 do
    begin
      Repo := ARepositoryList.Items[i] as TFPRepository;
      if IncludeRepositoryTypeForPackageKind(Repo.RepositoryType, APackageKind) then
        begin
          Result := repo.FindPackage(APackageName);
          if Assigned(Result) then
            Break;
        end;
    end;
end;

function TpkgFPpkg.SelectRemoteMirror: string;
var
  i,j : Integer;
  Bucket,
  BucketCnt : Integer;
  M : TFPMirror;
begin
  Result:='';
  M:=nil;
  if assigned(AvailableMirrors) then
   begin
     // Create array for selection
     BucketCnt:=0;
     for i:=0 to AvailableMirrors.Count-1 do
       inc(BucketCnt,AvailableMirrors[i].Weight);
     // Select random entry
     Bucket:=Random(BucketCnt);
     M:=nil;
     for i:=0 to AvailableMirrors.Count-1 do
       begin
         for j:=0 to AvailableMirrors[i].Weight-1 do
           begin
             if Bucket=0 then
               begin
                 M:=AvailableMirrors[i];
                 break;
               end;
             Dec(Bucket);
           end;
         if assigned(M) then
           break;
       end;
    end;
  if assigned(M) then
    begin
      log(llInfo,SLogSelectedMirror,[M.Name]);
      Result:=M.URL;
    end
  else
    Error(SErrFailedToSelectMirror);
end;

procedure TpkgFPpkg.EnterFindBrokenPackages;
begin
  Assert((FInsideFindBrokenPackages>0) or (FBrokenPackagesDictionary.Count=0));
  Inc(FInsideFindBrokenPackages)
end;

procedure TpkgFPpkg.LeaveFindBrokenpackages;
begin
  Assert(FInsideFindBrokenPackages>0);
  Dec(FInsideFindBrokenPackages);
  if FInsideFindBrokenPackages=0 then
    FBrokenPackagesDictionary.Clear;
end;

procedure TpkgFPpkg.ClearRepositories(ARepositoryList: TComponentList);
var
  i: Integer;
  Repo: TFPRepository;
begin
  for i := ARepositoryList.Count -1 downto 0 do
    begin
      Repo := ARepositoryList.Items[i] as TFPRepository;
      if Repo.Name <> 'Available' then
        begin
          ARepositoryList.Delete(i);
          Repo.Free;
        end;
    end;
end;

function TpkgFPpkg.PackageByName(APackageName: string; APackageKind: TpkgPackageKind): TFPPackage;
var
  ErrStr: string;
begin
  Result := FindPackage(APackageName, APackageKind);
  If Result=Nil then
    begin
      case APackageKind of
        pkgpkInstalled : ErrStr:=SErrMissingInstallPackage;
        pkgpkAvailable : ErrStr:=SErrMissingAvailablePackage;
        pkgpkBoth      : ErrStr:=SErrMissingPackage;
      end;
    Raise EPackage.CreateFmt(ErrStr,[APackageName]);
    end;
end;

function TpkgFPpkg.FindRepository(ARepositoryName: string): TFPRepository;
var
  i: Integer;
  Repo: TFPRepository;
begin
  Result := nil;
  for i := FRepositoryList.Count-1 downto 0 do
    begin
      Repo := FRepositoryList.Items[i] as TFPRepository;
      if Repo.RepositoryName = ARepositoryName then
        begin
          Result := Repo;
          Break;
        end;
    end;
end;

function TpkgFPpkg.RepositoryByName(ARepositoryName: string): TFPRepository;
begin
  Result := FindRepository(ARepositoryName);
  If Result=Nil then
    Raise EPackage.CreateFmt(SErrMissingInstallRepo,[ARepositoryName]);
end;

function TpkgFPpkg.GetInstallRepository(ASourcePackage: TFPPackage): TFPRepository;
var
  SourceRepository: TFPRepository;
  RepoName: string;
  i: Integer;
begin
  // Determine the repository to install a package into. See the
  // repositorylogics.dia file.
  pkgglobals.Log(llDebug, SLogDetermineInstallRepo, [ASourcePackage.GetDebugName]);
  RepoName := Options.CommandLineSection.InstallRepository;
  if RepoName <> '' then
    // If an install-repository is given on the command line, this overrides
    // everything.
    pkgglobals.Log(llDebug, SLogUseCommandLineRepo, [RepoName])
  else
    begin
      // The source-repository is already determined by the source-package, which
      // is a member of the source-repository.
      SourceRepository := ASourcePackage.Repository;
      Assert(Assigned(SourceRepository));
      Assert(SourceRepository.RepositoryType = fprtAvailable);

      // For now, skip the check for original sources of already installed packages.

      Assert(Assigned(SourceRepository.DefaultPackagesStructure));
      RepoName := SourceRepository.DefaultPackagesStructure.InstallRepositoryName;
      if RepoName<>'' then
        pkgglobals.Log(llDebug, SLogUseSourceRepoInstRepo, [RepoName, SourceRepository.RepositoryName])
      else
        begin
          RepoName := Options.GlobalSection.InstallRepository;
          if RepoName<>'' then
            pkgglobals.Log(llDebug, SLogUseConfigurationRepo, [RepoName])
          else
            begin
              for i := RepositoryList.Count-1 downto 0 do
                begin
                  if (RepositoryList[i] as TFPRepository).RepositoryType = fprtInstalled then
                    begin
                      Result := TFPRepository(RepositoryList[i]);
                      pkgglobals.Log(llDebug, SLogUseLastRepo, [Result.RepositoryName]);
                      Exit;
                    end;
                end;
              raise EPackage.Create(SErrNoInstallRepoAvailable);
            end;
        end;
    end;
  Result := RepositoryByName(RepoName);
end;

function TpkgFPpkg.DetermineSourcePackage(APackageName: String): TFPPackage;
begin
  Result := FindPackage(APackageName, pkgpkAvailable);
end;

function TpkgFPpkg.PackageLocalArchive(APackage: TFPPackage): String;
begin
  if APackage.Name=CurrentDirPackageName then
    Error(SErrNoPackageSpecified)
  else if APackage.Name=CmdLinePackageName then
    Result:=APackage.LocalFileName
  else
    Result:=Options.GlobalSection.ArchivesDir+APackage.FileName;
end;

procedure TpkgFPpkg.ScanInstalledPackagesForAvailablePackages;
var
  i: Integer;
  Repo, AvailableRepo: TFPRepository;
  AvailStruc: TFPOriginalSourcePackagesStructure;
begin
  for i := 0 to FRepositoryList.Count-1 do
    begin
      Repo := FRepositoryList.Items[i] as TFPRepository;
      if Repo.RepositoryType = fprtInstalled then
        begin
          AvailableRepo := TFPRepository.Create(Self);
          FRepositoryList.Add(AvailableRepo);
          AvailableRepo.RepositoryType := fprtAvailable;
          AvailableRepo.RepositoryName := Repo.RepositoryName + '_source';
          AvailableRepo.Description := Repo.Description + ' (original sources)';
          AvailStruc := TFPOriginalSourcePackagesStructure.Create(Self, Repo);
          AvailStruc.InitializeWithOptions(nil, FOptions, FCompilerOptions);
          AvailStruc.InstallRepositoryName := Repo.RepositoryName;
          AvailStruc.AddPackagesToRepository(AvailableRepo);
          AvailableRepo.DefaultPackagesStructure := AvailStruc;
        end;
    end;
end;

procedure TpkgFPpkg.CheckFPMakeDependencies;
var
  i : Integer;
  P,AvailP : TFPPackage;
  AvailVerStr : string;
  ReqVer : TFPVersion;
begin
  // Reset availability
  for i:=0 to high(FPMKUnitDeps) do
    FPMKUnitDeps[i].available:=false;
  // Not version check needed in Recovery mode, we always need to use
  // the internal bootstrap procedure
  if Options.CommandLineSection.RecoveryMode then
    exit;
  // Check for fpmkunit dependencies
  for i:=0 to high(FPMKUnitDeps) do
    begin
      P:=FPMakeRepoFindPackage(FPMKUnitDeps[i].package, pkgpkInstalled);
      if P<>nil then
        begin
          AvailP:=FindPackage(FPMKUnitDeps[i].package, pkgpkAvailable);
          if AvailP<>nil then
            AvailVerStr:=AvailP.Version.AsString
          else
            AvailVerStr:='<not available>';
          ReqVer:=TFPVersion.Create;
          try
            ReqVer.AsString:=FPMKUnitDeps[i].ReqVer;
            log(llDebug,SLogFPMKUnitDepVersion,[P.Name,ReqVer.AsString,P.Version.AsString,AvailVerStr]);
            if ReqVer.CompareVersion(P.Version)<=0 then
              FPMKUnitDeps[i].available:=true
            else
              log(llDebug,SLogFPMKUnitDepTooOld,[FPMKUnitDeps[i].package]);
          finally
            ReqVer.Free;
          end;
        end
      else
        log(llDebug,SLogFPMKUnitDepTooOld,[FPMKUnitDeps[i].package]);
    end;
end;

function TpkgFPpkg.FindBrokenPackages(SL: TStrings): Boolean;
var
  i,j,k : integer;
  P : TFPPackage;
  s : string;
  Repo: TFPRepository;
begin
  SL.Clear;
  EnterFindBrokenPackages;
  try
    for i:=0 to RepositoryList.Count-1 do
      begin
        Repo := TFPRepository(RepositoryList[i]);
        if Repo.RepositoryType = fprtInstalled then
          begin
            for j := 0 to Repo.PackageCount-1 do
              begin
                P := Repo.Packages[j];
                if (P = FindPackage(P.Name, pkgpkInstalled)) and PackageIsBroken(P, s, nil) then
                  begin
                    if P.IsFPMakeAddIn then
                      // Make sure that FPMakeAddIn's are fixed first, so
                      // as much packages are compiled with them.
                      SL.Insert(0, P.Name)
                    else
                      SL.Add(P.Name);
                  end;
              end;
          end;
      end;
  finally
    LeaveFindBrokenpackages;
  end;
  Result:=(SL.Count>0);
end;

function TpkgFPpkg.PackageBuildPath(APackage: TFPPackage): String;
begin
  if (APackage.Name=CmdLinePackageName) or (APackage.Name=URLPackageName) then
    Result:=Options.GlobalSection.BuildDir+ChangeFileExt(ExtractFileName(APackage.LocalFileName),'')
  else if Assigned(APackage.PackagesStructure) and (APackage.PackagesStructure.GetBuildPathDirectory(APackage)<>'') then
    Result:=APackage.PackagesStructure.GetBuildPathDirectory(APackage)
  else
    Result:=Options.GlobalSection.BuildDir+APackage.Name;
end;

function TpkgFPpkg.GetRemoteRepositoryURL(const AFileName: string): string;
begin
  if FCurrentRemoteRepositoryURL='' then
    begin
      if Options.GlobalSection.RemoteRepository='auto' then
        FCurrentRemoteRepositoryURL:=SelectRemoteMirror
      else
        FCurrentRemoteRepositoryURL:=Options.GlobalSection.RemoteRepository;
    end;
  result := FCurrentRemoteRepositoryURL;
  if result <> '' then
    begin
      if result[length(result)]<>'/' then
        result := result + '/';
      Result:=Result+CompilerOptions.CompilerVersion+'/'+AFileName;
    end;
end;

function TpkgFPpkg.PackageRemoteArchive(APackage: TFPPackage): String;
begin
  if APackage.Name=CurrentDirPackageName then
    Error(SErrNoPackageSpecified)
  else if APackage.Name=CmdLinePackageName then
    Error(SErrPackageIsLocal);
  if APackage.DownloadURL<>'' then
    Result:=APackage.DownloadURL
  else
    Result:=GetRemoteRepositoryURL(APackage.FileName);
end;

function TpkgFPpkg.GetConfigurationFilename: string;
begin
  Result := FConfigurationFilename;
end;

end.

