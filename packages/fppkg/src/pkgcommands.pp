unit pkgcommands;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, pkghandler, fpmkunit;

implementation

uses
  zipper,
  pkgmessages,
  pkgglobals,
  pkgoptions,
  pkgdownload,
  pkgrepos,
  pkgFppkg,
  fpxmlrep,
  fprepos;

type
  { TCommandAddConfig }

  TCommandAddConfig = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandUpdate }

  TCommandUpdate = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandListPackages }

  TCommandListPackages = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandScanPackages }

  TCommandScanPackages = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandDownload }

  TCommandDownload = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandUnzip }

  TCommandUnzip = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandCompile }

  TCommandCompile = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandBuild }

  TCommandBuild = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandInstall }

  TCommandInstall = Class(TPackagehandler)
  protected
    function ForceInstall: Boolean; virtual;
  Public
    function Execute: Boolean;override;
  end;

  { TCommandInstallForced }

  TCommandInstallForced = Class(TCommandInstall)
  protected
    function ForceInstall: Boolean; override;
  end;


  { TCommandUnInstall }

  TCommandUnInstall = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandClean }

  TCommandClean = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandArchive }

  TCommandArchive = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandInstallDependencies }

  TCommandInstallDependencies = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandFixBroken }

  TCommandFixBroken = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandListSettings }

  TCommandListSettings = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

  { TCommandInfo }

  TCommandInfo = Class(TPackagehandler)
  Public
    function Execute: Boolean;override;
  end;

var
  DependenciesDepth: integer;

{ TCommandInstallForced }

function TCommandInstallForced.ForceInstall: Boolean;
begin
  Result := True;
end;

{ TCommandInfo }

function TCommandInfo.Execute: Boolean;
var
  P : TFPPackage;
  S : string;
  I : Integer;
begin
  Result := True;
  if PackageName='' then
    Error(SErrNoPackageSpecified);
  P:=PackageManager.PackageByName(PackageName, pkgpkAvailable);

  log(llProgress,SLogPackageInfoName,[P.Name]);
  S := P.Email;
  if S <> '' then
    S := '<' + S +'>';
  log(llProgress,SLogPackageInfoAuthor,[P.Author, S]);
  log(llProgress,SLogPackageInfoVersion,[P.Version.AsString]);
  log(llProgress,SLogPackageInfoCategory,[P.Category]);
  log(llProgress,SLogPackageInfoWebsite,[P.HomepageURL]);
  log(llProgress,SLogPackageInfoLicense,[P.License]);

  log(llProgress,SLogPackageInfoOSes,[OSesToString(P.OSes)]);
  log(llProgress,SLogPackageInfoCPUs,[CPUSToString(P.CPUs)]);

  log(llProgress,SLogPackageInfoDescription1);
  log(llProgress,SLogPackageInfoDescription2,[P.Description]);

  if P.Dependencies.Count>0 then
    begin
      log(llProgress,SLogPackageInfoDependencies1,[]);
      for I := 0 to P.Dependencies.Count-1 do
        begin
          if not P.Dependencies[I].MinVersion.Empty then
            S := '('+P.Dependencies[I].MinVersion.AsString+')'
          else
            S := '';
          log(llProgress,SLogPackageInfoDependencies2,[P.Dependencies[I].PackageName,S]);
        end;
    end;
end;

{ TCommandUnInstall }

function TCommandUnInstall.Execute: Boolean;
var
  AvailP: TFPPackage;
begin
  Result := False;
  if PackageName<>'' then
    begin
      if (PackageName=CmdLinePackageName) then
        begin
          ExecuteAction(PackageName,'unzip');
        end
      else if (PackageName<>CurrentDirPackageName) then
        begin
          AvailP:=PackageManager.FindPackage(PackageName, pkgpkAvailable);
          if Assigned(AvailP) then
            begin
              if AvailP.PackagesStructure.UnzipBeforeUse then
                ExecuteAction(PackageName,'unzip');
            end;
        end;
    end;
  Result := ExecuteAction(PackageName,'fpmakeuninstall');
end;

{ TCommandListSettings }

function TCommandListSettings.Execute: Boolean;
begin
  Result := True;
  PackageManager.Options.LogValues(llProgress);
  PackageManager.CompilerOptions.LogValues(llProgress,'');
  PackageManager.FPMakeCompilerOptions.LogValues(llProgress,'fpmake-building ');
end;


function TCommandAddConfig.Execute: Boolean;
begin
  Result := False;
{
  Log(llInfo,SLogGeneratingCompilerConfig,[S]);
  Options.InitCompilerDefaults(Args[2]);
  Options.SaveCompilerToFile(S);
}
end;


function TCommandUpdate.Execute: Boolean;
var
  PackagesURL :  String;
begin
  Result := True;
  // Download and load mirrors.xml
  // This can be skipped when a custom RemoteRepository is configured
  if (PackageManager.Options.GlobalSection.RemoteMirrorsURL<>'') and
     (PackageManager.Options.GlobalSection.RemoteRepository='auto') then
    begin
      Log(llCommands,SLogDownloading,[PackageManager.Options.GlobalSection.RemoteMirrorsURL,PackageManager.Options.GlobalSection.LocalMirrorsFile]);
      Result := DownloadFile(PackageManager.Options.GlobalSection.RemoteMirrorsURL,PackageManager.Options.GlobalSection.LocalMirrorsFile, PackageManager);
      PackageManager.LoadLocalAvailableMirrors;
    end;
  // Download packages.xml
  PackagesURL:=PackageManager.GetRemoteRepositoryURL(PackagesFileName);
  Log(llCommands,SLogDownloading,[PackagesURL,PackageManager.Options.GlobalSection.LocalPackagesFile]);
  Result := Result and DownloadFile(PackagesURL,PackageManager.Options.GlobalSection.LocalPackagesFile,PackageManager);
  // Read the repository again
  PackageManager.ScanAvailablePackages;
  // no need to log errors again
  PackageManager.ScanPackages;
end;


function TCommandListPackages.Execute: Boolean;
begin
  Result := True;
  ListPackages(PackageManager.Options.CommandLineSection.ShowLocation);
end;


function TCommandScanPackages.Execute: Boolean;
begin
  Result := True;
  { nothing, already handled in fppkg.pp as special case
    before the local fppkg directory is processed }
end;


function TCommandDownload.Execute: Boolean;
var
  P : TFPPackage;
begin
  Result := False;
  if PackageName='' then
    Error(SErrNoPackageSpecified);
  P:=PackageManager.PackageByName(PackageName, pkgpkAvailable);
  if not FileExists(PackageManager.PackageLocalArchive(P)) then
    Result := ExecuteAction(PackageName,'downloadpackage')
  else
    Result := True;
end;


function TCommandUnzip.Execute: Boolean;
Var
  BuildDir : string;
  ArchiveFile : String;
  P : TFPPackage;
begin
  Result := False;
  if PackageName='' then
    begin
      Error(SErrNoPackageSpecified);
      Exit;
    end;
  P:=PackageManager.PackageByName(PackageName, pkgpkAvailable);
  BuildDir:=PackageManager.PackageBuildPath(P);
  ArchiveFile:=PackageManager.PackageLocalArchive(P);
  if not FileExists(ArchiveFile) then
    if not ExecuteAction(PackageName,'downloadpackage') then
      Exit;
  { Create builddir, remove it first if needed }
  if DirectoryExists(BuildDir) then
    DeleteDir(BuildDir);
  ForceDirectories(BuildDir);
  SetCurrentDir(BuildDir);
  { Unzip Archive }
  With TUnZipper.Create do
    try
      Log(llCommands,SLogUnzippping,[ArchiveFile]);
      OutputPath:=PackageManager.PackageBuildPath(P);
      UnZipAllFiles(ArchiveFile);
      Result := True;
    Finally
      Free;
    end;
end;


function TCommandCompile.Execute: Boolean;
begin
  if PackageName<>'' then
    begin
      // For local files we need the information inside the zip to get the
      // dependencies
      if (PackageName=CmdLinePackageName) or (PackageName=URLPackageName) then
        begin
          if not ExecuteAction(PackageName,'unzip') then
            Exit;
          if not ExecuteAction(PackageName,'installdependencies') then
            Exit;
        end
      else
        if (PackageName=CurrentDirPackageName) then
          begin
            if not ExecuteAction(PackageName,'installdependencies') then
              Exit;
          end
      else
        begin
          if not ExecuteAction(PackageName,'installdependencies') then
            Exit;
          if not ExecuteAction(PackageName,'unzip') then
            Exit;
        end;
    end;
  Result := ExecuteAction(PackageName,'fpmakecompile');
end;


function TCommandBuild.Execute: Boolean;
var
  P: TFPPackage;
begin
  Result := False;
  if PackageName<>'' then
    begin
      // For local files we need the information inside the zip to get the
      // dependencies
      if (PackageName=CmdLinePackageName) or (PackageName=URLPackageName) then
        begin
          if not ExecuteAction(PackageName,'unzip') then
            Exit;
          if not ExecuteAction(PackageName,'installdependencies') then
            Exit;
        end
      else
        if (PackageName=CurrentDirPackageName) then
          begin
            if not ExecuteAction(PackageName,'installdependencies') then
              Exit;
          end
      else
        begin
          P:=PackageManager.FindPackage(PackageName, pkgpkAvailable);
          if not Assigned(P) then
            begin
              Error(Format(SErrPackageNotAvailable, [PackageName]));
              Exit;
            end;

          if P.PackagesStructure.UnzipBeforeUse then
            if not ExecuteAction(PackageName,'unzip') then
              Exit;
          if not ExecuteAction(PackageName,'installdependencies') then
            Exit;
        end;
    end;
  Result := ExecuteAction(PackageName,'fpmakebuild');
end;

function TCommandInstall.ForceInstall: Boolean;
begin
  Result := False;
end;

function TCommandInstall.Execute: Boolean;

var
  S : String;
  P   : TFPPackage;
  InstallRepo: TFPRepository;

  function GetFpmFilename(APackage: TFPPackage): string;
  var
    ConfFile: string;
  begin
    Result := '';
    if Assigned(InstallRepo.DefaultPackagesStructure) then
      begin
        ConfFile := InstallRepo.DefaultPackagesStructure.GetConfigFileForPackage(APackage);
        if not FileExistsLog(ConfFile) then
          begin
            // If there is no fpm-file, search for an (obsolete, pre-2.7.x)
            // fpunits.cfg-file
            ConfFile := IncludeTrailingPathDelimiter(Result)+APackage.Name+PathDelim+UnitConfigFileName;
            if FileExistsLog(ConfFile) then
              Result := ConfFile;
          end
        else
          Result := ConfFile;
      end;
  end;

var
  UFN : String;
  AvailPackage: TFPPackage;
begin
  if PackageName<>'' then
    begin
      Result := False;
      if not ExecuteAction(PackageName,'build') then
        Exit;

      AvailPackage := PackageManager.DetermineSourcePackage(PackageName);
      InstallRepo := PackageManager.GetInstallRepository(AvailPackage);   // AvailPackage can be nil! currentdir - fixbroken ||  embweb - install-req
      case InstallRepo.DefaultPackagesStructure.IsInstallationNeeded(AvailPackage) of
        fpinInstallationNeeded:
          begin
            if Assigned(AvailPackage) then
              S := AvailPackage.GetDebugName
            else
              S := PackageName;
            log(llDebug,SDbgPackageInstallRequired,[S, InstallRepo.RepositoryName]);
            if not ExecuteAction(PackageName,'fpmakeinstall') then
              Exit;
          end;
        fpinInstallationImpossible:
          begin
            Error(SErrInstallationImpossible,[PackageName, InstallRepo.RepositoryName]);
            Exit;
          end
        else if ForceInstall then
          begin
            log(llDebug,SDbgForcePackageInstall,[PackageName]);
            if not ExecuteAction(PackageName,'fpmakeinstall') then
              Exit;
          end;
      end;

      if (PackageName=CmdLinePackageName) or (PackageName=CurrentDirPackageName) or
         (PackageName=URLPackageName) then
        begin
          // Load package name from manifest
          if not FileExists(ManifestFileName) then
            if not ExecuteAction(PackageName,'fpmakemanifest') then
              Exit;
          P:=LoadManifestFromFile(ManifestFileName);
          S:=P.Name;
          FreeAndNil(P);
        end
      else
        S:=PackageName;

      if Assigned(InstallRepo) then
        begin
          P := InstallRepo.FindPackage(S);
          if not Assigned(P) then
            P := InstallRepo.AddPackage(S);
          if Assigned(P) then
            begin
              UFN:=GetFpmFilename(P);
              if UFN<>'' then
                begin
                  P.LoadUnitConfigFromFile(UFN);
                  if P.IsFPMakeAddIn then
                    AddFPMakeAddIn(P);
                end;
            end;
        end;
      Result := True;
    end
  else
    Result := ExecuteAction(PackageName,'fpmakeinstall');
end;


function TCommandClean.Execute: Boolean;
begin
  Result := ExecuteAction(PackageName,'fpmakeclean');
end;


function TCommandArchive.Execute: Boolean;
begin
  Result := ExecuteAction(PackageName,'fpmakearchive');
end;


function TCommandInstallDependencies.Execute: Boolean;

  function PackageVersionStr(APackage: TFPPackage): string;
  begin
    if Assigned(APackage) then
      Result := APackage.Version.AsString
    else
      Result := 'N/A';
  end;

var
  i : Integer;
  MissingDependency,
  D : TFPDependency;
  P,
  InstalledP,
  AvailP : TFPPackage;
  PackNr: integer;
  ManifestPackages : TFPPackages;
  X : TFPXMLRepositoryHandler;
  L : TStringList;
  s : string;
  status : string;
begin
  Result := False;
  if PackageName='' then
    Error(SErrNoPackageSpecified);
  ManifestPackages:=nil;
  // Load dependencies for local packages
  if (PackageName=CmdLinePackageName) or (PackageName=CurrentDirPackageName) or
     (PackageName=URLPackageName) then
    begin
      ExecuteAction(PackageName,'fpmakemanifest');
      ManifestPackages:=TFPPackages.Create(TFPPackage);
      X:=TFPXMLRepositoryHandler.Create;
      try
        X.LoadFromXml(ManifestPackages,ManifestFileName);
      finally
        X.Free;
      end;
      if ManifestPackages.Count>0 then
        begin
          PackNr:=0;
          P := ManifestPackages[PackNr];
        end
      else
        begin
          ManifestPackages.Free;
          Error(SErrManifestNoSinglePackage,[ManifestFileName]);
          Exit;
        end;
    end
  else
    P:=PackageManager.PackageByName(PackageName, pkgpkBoth);

  MissingDependency:=nil;
  while assigned(P) do
    begin
      // Find and List dependencies
      L:=TStringList.Create;

      if not ((PackageManager.CompilerOptions.CompilerOS in P.OSes) and (PackageManager.CompilerOptions.CompilerCPU in P.CPUs)) then
        Log(llDebug,SDbgPackageDependencyOtherTarget,[P.Name,MakeTargetString(PackageManager.CompilerOptions.CompilerCPU,PackageManager.CompilerOptions.CompilerOS)])
      else
        for i:=0 to P.Dependencies.Count-1 do
          begin
            D:=P.Dependencies[i];
            if not ((PackageManager.CompilerOptions.CompilerOS in D.OSes) and (PackageManager.CompilerOptions.CompilerCPU in D.CPUs)) then
              Log(llDebug,SDbgPackageDependencyOtherTarget,[D.PackageName,MakeTargetString(PackageManager.CompilerOptions.CompilerCPU,PackageManager.CompilerOptions.CompilerOS)])
            // Skip dependencies that are available within the fpmake-file itself
            else if not (assigned(ManifestPackages) and assigned(ManifestPackages.FindPackage(D.PackageName))) then
              begin
                AvailP := nil;
                InstalledP:=PackageManager.FindPackage(D.PackageName, pkgpkInstalled);
                // Need installation?
                if not assigned(InstalledP) or
                   (InstalledP.Version.CompareVersion(D.MinVersion)<0) then
                  begin
                    AvailP:=PackageManager.FindPackage(D.PackageName, pkgpkAvailable);
                    if not assigned(AvailP) or
                       (AvailP.Version.CompareVersion(D.MinVersion)<0) then
                      begin
                        status:='Not Available!';
                        MissingDependency:=D;
                      end
                    else
                      begin
                        status:='Updating';
                        L.Add(D.PackageName);
                      end;
                  end
                else
                  begin
                    if PackageManager.PackageIsBroken(InstalledP, s, nil) then
                      begin
                        status:='Broken, recompiling';
                        L.Add(D.PackageName);
                      end
                    else
                      status:='OK';
                  end;
                Log(llInfo,SLogPackageDependency,
                    [D.PackageName,D.MinVersion.AsString,PackageVersionStr(InstalledP),
                     PackageVersionStr(AvailP),status])
              end
          end;
      if assigned(ManifestPackages) and (PackNr<ManifestPackages.Count-1)  then
        begin
          inc(PackNr);
          P := ManifestPackages[PackNr]
        end
      else
        p := nil;
    end;
  // Give error on first missing dependency
  if assigned(MissingDependency) then
    Error(SErrNoPackageAvailable,[MissingDependency.PackageName,MissingDependency.MinVersion.AsString]);
  // Install needed updates
  if L.Count > 0 then
    begin
      if DependenciesDepth=0 then
        pkgglobals.Log(llProgress,SProgrInstallDependencies);
      inc(DependenciesDepth);

      for i:=0 to L.Count-1 do
        if not ExecuteAction(L[i],'install-req') then
          Exit;

      dec(DependenciesDepth);
      if DependenciesDepth=0 then
        pkgglobals.Log(llProgress,SProgrDependenciesInstalled);
    end;
  FreeAndNil(L);
  if assigned(ManifestPackages) then
    ManifestPackages.Free;
  Result := True;
end;


function TCommandFixBroken.Execute: Boolean;
var
  i : integer;
  SL : TStringList;
  BreakLoop : Boolean;
  Reason: string;
begin
  Result := False;
  SL:=TStringList.Create;
  BreakLoop := false;
  repeat
    PackageManager.FindBrokenPackages(SL);
    if SL.Count=0 then
      break;
    pkgglobals.Log(llProgress,SProgrReinstallDependent);
    for i:=0 to SL.Count-1 do
      begin
        if not ExecuteAction(SL[i],'build') then
          Exit;
        if not ExecuteAction(SL[i],'install-req') then
          Exit;
        if PackageManager.PackageIsBroken(PackageManager.PackageByName(SL[i], pkgpkInstalled), Reason, nil) then
          begin
            BreakLoop := true;
            pkgglobals.Log(llWarning, SWarnBrokenAfterReinstall, [SL[i], Reason]);
          end;
      end;
  until BreakLoop;
  Result := True;
  FreeAndNil(SL);
end;


initialization
  DependenciesDepth:=0;
  RegisterPkgHandler('update',TCommandUpdate);
  RegisterPkgHandler('list',TCommandListPackages);
  RegisterPkgHandler('scan',TCommandScanPackages);
  RegisterPkgHandler('download',TCommandDownload);
  RegisterPkgHandler('unzip',TCommandUnzip);
  RegisterPkgHandler('compile',TCommandCompile);
  RegisterPkgHandler('build',TCommandBuild);
  RegisterPkgHandler('install',TCommandInstallForced);
  RegisterPkgHandler('install-req',TCommandInstall);
  RegisterPkgHandler('uninstall',TCommandUnInstall);
  RegisterPkgHandler('clean',TCommandClean);
  RegisterPkgHandler('archive',TCommandArchive);
  RegisterPkgHandler('installdependencies',TCommandInstallDependencies);
  RegisterPkgHandler('fixbroken',TCommandFixBroken);
  RegisterPkgHandler('listsettings',TCommandListSettings);
  RegisterPkgHandler('info',TCommandInfo);
end.
